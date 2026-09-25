from http import HTTPStatus
from typing import Any, cast
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.error_envelope import ErrorEnvelope
from ...types import UNSET, Response, Unset


def _get_kwargs(
    id: str,
    name: str,
    *,
    offset: int | Unset = 0,
    limit: int | Unset = 262144,
) -> dict[str, Any]:

    params: dict[str, Any] = {}

    params["offset"] = offset

    params["limit"] = limit

    params = {k: v for k, v in params.items() if v is not UNSET and v is not None}

    _kwargs: dict[str, Any] = {
        "method": "get",
        "url": "/v1/recorder/dumps/{id}/files/{name}".format(
            id=quote(str(id), safe=""),
            name=quote(str(name), safe=""),
        ),
        "params": params,
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | str | None:
    if response.status_code == 200:
        response_200 = cast(str, response.json())
        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ErrorEnvelope | str]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    id: str,
    name: str,
    *,
    client: AuthenticatedClient | Client,
    offset: int | Unset = 0,
    limit: int | Unset = 262144,
) -> Response[ErrorEnvelope | str]:
    """Read one window of one file of one dump

     Answers with the file's OWN BYTES, not with a JSON envelope: application/x-ndjson for a .jsonl file
    and application/json for a .json one. A failure is still the JSON error envelope, so branch on the
    response content type and not on the status alone. name must be one path segment and must be one the
    manifest's files list carries; nothing else is served, and a dump directory is flat, so a
    namespace's copy is statelog.arm.a.jsonl rather than a path. The window is the byte range
    offset..offset+limit, bounded at 262144 bytes per call and not trimmed, so concatenated windows are
    the file itself. X-Dump-File-Offset, X-Dump-File-Total-Bytes and X-Dump-File-Eof say where the
    window sits; the next call starts at the offset plus this response's Content-Length. An offset past
    the end of the file is invalid.

    Args:
        id (str):
        name (str):
        offset (int | Unset):  Default: 0.
        limit (int | Unset):  Default: 262144.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | str]
    """

    kwargs = _get_kwargs(
        id=id,
        name=name,
        offset=offset,
        limit=limit,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    id: str,
    name: str,
    *,
    client: AuthenticatedClient | Client,
    offset: int | Unset = 0,
    limit: int | Unset = 262144,
) -> ErrorEnvelope | str | None:
    """Read one window of one file of one dump

     Answers with the file's OWN BYTES, not with a JSON envelope: application/x-ndjson for a .jsonl file
    and application/json for a .json one. A failure is still the JSON error envelope, so branch on the
    response content type and not on the status alone. name must be one path segment and must be one the
    manifest's files list carries; nothing else is served, and a dump directory is flat, so a
    namespace's copy is statelog.arm.a.jsonl rather than a path. The window is the byte range
    offset..offset+limit, bounded at 262144 bytes per call and not trimmed, so concatenated windows are
    the file itself. X-Dump-File-Offset, X-Dump-File-Total-Bytes and X-Dump-File-Eof say where the
    window sits; the next call starts at the offset plus this response's Content-Length. An offset past
    the end of the file is invalid.

    Args:
        id (str):
        name (str):
        offset (int | Unset):  Default: 0.
        limit (int | Unset):  Default: 262144.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | str
    """

    return sync_detailed(
        id=id,
        name=name,
        client=client,
        offset=offset,
        limit=limit,
    ).parsed


async def asyncio_detailed(
    id: str,
    name: str,
    *,
    client: AuthenticatedClient | Client,
    offset: int | Unset = 0,
    limit: int | Unset = 262144,
) -> Response[ErrorEnvelope | str]:
    """Read one window of one file of one dump

     Answers with the file's OWN BYTES, not with a JSON envelope: application/x-ndjson for a .jsonl file
    and application/json for a .json one. A failure is still the JSON error envelope, so branch on the
    response content type and not on the status alone. name must be one path segment and must be one the
    manifest's files list carries; nothing else is served, and a dump directory is flat, so a
    namespace's copy is statelog.arm.a.jsonl rather than a path. The window is the byte range
    offset..offset+limit, bounded at 262144 bytes per call and not trimmed, so concatenated windows are
    the file itself. X-Dump-File-Offset, X-Dump-File-Total-Bytes and X-Dump-File-Eof say where the
    window sits; the next call starts at the offset plus this response's Content-Length. An offset past
    the end of the file is invalid.

    Args:
        id (str):
        name (str):
        offset (int | Unset):  Default: 0.
        limit (int | Unset):  Default: 262144.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | str]
    """

    kwargs = _get_kwargs(
        id=id,
        name=name,
        offset=offset,
        limit=limit,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    id: str,
    name: str,
    *,
    client: AuthenticatedClient | Client,
    offset: int | Unset = 0,
    limit: int | Unset = 262144,
) -> ErrorEnvelope | str | None:
    """Read one window of one file of one dump

     Answers with the file's OWN BYTES, not with a JSON envelope: application/x-ndjson for a .jsonl file
    and application/json for a .json one. A failure is still the JSON error envelope, so branch on the
    response content type and not on the status alone. name must be one path segment and must be one the
    manifest's files list carries; nothing else is served, and a dump directory is flat, so a
    namespace's copy is statelog.arm.a.jsonl rather than a path. The window is the byte range
    offset..offset+limit, bounded at 262144 bytes per call and not trimmed, so concatenated windows are
    the file itself. X-Dump-File-Offset, X-Dump-File-Total-Bytes and X-Dump-File-Eof say where the
    window sits; the next call starts at the offset plus this response's Content-Length. An offset past
    the end of the file is invalid.

    Args:
        id (str):
        name (str):
        offset (int | Unset):  Default: 0.
        limit (int | Unset):  Default: 262144.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | str
    """

    return (
        await asyncio_detailed(
            id=id,
            name=name,
            client=client,
            offset=offset,
            limit=limit,
        )
    ).parsed
