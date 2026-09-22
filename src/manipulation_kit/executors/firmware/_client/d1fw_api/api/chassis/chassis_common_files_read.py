from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_common_file_read import ChassisCommonFileRead
from ...models.chassis_common_files_read_response_200 import (
    ChassisCommonFilesReadResponse200,
)
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    *,
    body: ChassisCommonFileRead,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/chassis/common_files/read",
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisCommonFilesReadResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisCommonFilesReadResponse200.from_dict(response.json())

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
) -> Response[ChassisCommonFilesReadResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisCommonFileRead,
) -> Response[ChassisCommonFilesReadResponse200 | ErrorEnvelope]:
    """Read opaque common-file storage

     Explicit validated type and basename; no inferred type inventory. Canonical base64; 512 KiB raw file
    cap and 1 MiB list cap. Marked downloads retain opaque bytes including JSON code fields.
    Replace/delete require fresh exact content revision. Upload requires successful fresh listing
    absence; this is optimistic regular-file absence, not namespace/path absence, and can replace an
    empty same-name directory. Initial list code 735 blocks namespace bootstrap. Single non-atomic send
    and exact readback; no retry/rollback. Verified delete means absent from a successful regular-file
    listing, never physical path absence; last-file code 735 is outcome_unknown. No batch/apply/restart
    or physical completion claim. Consumers must use inert text preview or download, never execute
    stored HTML/JS.

    Args:
        body (ChassisCommonFileRead): Exact basename within the explicit type; never a filesystem
            path.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisCommonFilesReadResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisCommonFileRead,
) -> ChassisCommonFilesReadResponse200 | ErrorEnvelope | None:
    """Read opaque common-file storage

     Explicit validated type and basename; no inferred type inventory. Canonical base64; 512 KiB raw file
    cap and 1 MiB list cap. Marked downloads retain opaque bytes including JSON code fields.
    Replace/delete require fresh exact content revision. Upload requires successful fresh listing
    absence; this is optimistic regular-file absence, not namespace/path absence, and can replace an
    empty same-name directory. Initial list code 735 blocks namespace bootstrap. Single non-atomic send
    and exact readback; no retry/rollback. Verified delete means absent from a successful regular-file
    listing, never physical path absence; last-file code 735 is outcome_unknown. No batch/apply/restart
    or physical completion claim. Consumers must use inert text preview or download, never execute
    stored HTML/JS.

    Args:
        body (ChassisCommonFileRead): Exact basename within the explicit type; never a filesystem
            path.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisCommonFilesReadResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisCommonFileRead,
) -> Response[ChassisCommonFilesReadResponse200 | ErrorEnvelope]:
    """Read opaque common-file storage

     Explicit validated type and basename; no inferred type inventory. Canonical base64; 512 KiB raw file
    cap and 1 MiB list cap. Marked downloads retain opaque bytes including JSON code fields.
    Replace/delete require fresh exact content revision. Upload requires successful fresh listing
    absence; this is optimistic regular-file absence, not namespace/path absence, and can replace an
    empty same-name directory. Initial list code 735 blocks namespace bootstrap. Single non-atomic send
    and exact readback; no retry/rollback. Verified delete means absent from a successful regular-file
    listing, never physical path absence; last-file code 735 is outcome_unknown. No batch/apply/restart
    or physical completion claim. Consumers must use inert text preview or download, never execute
    stored HTML/JS.

    Args:
        body (ChassisCommonFileRead): Exact basename within the explicit type; never a filesystem
            path.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisCommonFilesReadResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisCommonFileRead,
) -> ChassisCommonFilesReadResponse200 | ErrorEnvelope | None:
    """Read opaque common-file storage

     Explicit validated type and basename; no inferred type inventory. Canonical base64; 512 KiB raw file
    cap and 1 MiB list cap. Marked downloads retain opaque bytes including JSON code fields.
    Replace/delete require fresh exact content revision. Upload requires successful fresh listing
    absence; this is optimistic regular-file absence, not namespace/path absence, and can replace an
    empty same-name directory. Initial list code 735 blocks namespace bootstrap. Single non-atomic send
    and exact readback; no retry/rollback. Verified delete means absent from a successful regular-file
    listing, never physical path absence; last-file code 735 is outcome_unknown. No batch/apply/restart
    or physical completion claim. Consumers must use inert text preview or download, never execute
    stored HTML/JS.

    Args:
        body (ChassisCommonFileRead): Exact basename within the explicit type; never a filesystem
            path.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisCommonFilesReadResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
