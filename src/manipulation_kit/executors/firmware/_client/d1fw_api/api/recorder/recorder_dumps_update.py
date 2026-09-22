from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.dump_update import DumpUpdate
from ...models.error_envelope import ErrorEnvelope
from ...models.recorder_dumps_update_response_200 import RecorderDumpsUpdateResponse200
from ...types import Response


def _get_kwargs(
    id: str,
    *,
    body: DumpUpdate,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "patch",
        "url": "/v1/recorder/dumps/{id}".format(
            id=quote(str(id), safe=""),
        ),
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | RecorderDumpsUpdateResponse200 | None:
    if response.status_code == 200:
        response_200 = RecorderDumpsUpdateResponse200.from_dict(response.json())

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
) -> Response[ErrorEnvelope | RecorderDumpsUpdateResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    id: str,
    *,
    client: AuthenticatedClient | Client,
    body: DumpUpdate,
) -> Response[ErrorEnvelope | RecorderDumpsUpdateResponse200]:
    """Set a dump's note or its pinned flag

     A partial update: omitting a field leaves it as it is. An EMPTY note clears it, which is how a note
    is removed without a second field to say so. The body carries no id -- which dump is the path -- and
    unknown fields are refused. A pinned dump is never evicted automatically, and the manifest on disk
    is rewritten atomically before the answer is sent.

    Args:
        id (str):
        body (DumpUpdate): `PATCH /v1/recorder/dumps/{id}`: the fields of one dump an operator may
            change.

            Which dump is not in the body: it is the `{id}` of the path, because a
            dump is a resource this daemon owns and a PATCH addresses the resource it
            is sent to. A body that could name a second dump would be a body that can
            disagree with the path.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | RecorderDumpsUpdateResponse200]
    """

    kwargs = _get_kwargs(
        id=id,
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    id: str,
    *,
    client: AuthenticatedClient | Client,
    body: DumpUpdate,
) -> ErrorEnvelope | RecorderDumpsUpdateResponse200 | None:
    """Set a dump's note or its pinned flag

     A partial update: omitting a field leaves it as it is. An EMPTY note clears it, which is how a note
    is removed without a second field to say so. The body carries no id -- which dump is the path -- and
    unknown fields are refused. A pinned dump is never evicted automatically, and the manifest on disk
    is rewritten atomically before the answer is sent.

    Args:
        id (str):
        body (DumpUpdate): `PATCH /v1/recorder/dumps/{id}`: the fields of one dump an operator may
            change.

            Which dump is not in the body: it is the `{id}` of the path, because a
            dump is a resource this daemon owns and a PATCH addresses the resource it
            is sent to. A body that could name a second dump would be a body that can
            disagree with the path.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | RecorderDumpsUpdateResponse200
    """

    return sync_detailed(
        id=id,
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    id: str,
    *,
    client: AuthenticatedClient | Client,
    body: DumpUpdate,
) -> Response[ErrorEnvelope | RecorderDumpsUpdateResponse200]:
    """Set a dump's note or its pinned flag

     A partial update: omitting a field leaves it as it is. An EMPTY note clears it, which is how a note
    is removed without a second field to say so. The body carries no id -- which dump is the path -- and
    unknown fields are refused. A pinned dump is never evicted automatically, and the manifest on disk
    is rewritten atomically before the answer is sent.

    Args:
        id (str):
        body (DumpUpdate): `PATCH /v1/recorder/dumps/{id}`: the fields of one dump an operator may
            change.

            Which dump is not in the body: it is the `{id}` of the path, because a
            dump is a resource this daemon owns and a PATCH addresses the resource it
            is sent to. A body that could name a second dump would be a body that can
            disagree with the path.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | RecorderDumpsUpdateResponse200]
    """

    kwargs = _get_kwargs(
        id=id,
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    id: str,
    *,
    client: AuthenticatedClient | Client,
    body: DumpUpdate,
) -> ErrorEnvelope | RecorderDumpsUpdateResponse200 | None:
    """Set a dump's note or its pinned flag

     A partial update: omitting a field leaves it as it is. An EMPTY note clears it, which is how a note
    is removed without a second field to say so. The body carries no id -- which dump is the path -- and
    unknown fields are refused. A pinned dump is never evicted automatically, and the manifest on disk
    is rewritten atomically before the answer is sent.

    Args:
        id (str):
        body (DumpUpdate): `PATCH /v1/recorder/dumps/{id}`: the fields of one dump an operator may
            change.

            Which dump is not in the body: it is the `{id}` of the path, because a
            dump is a resource this daemon owns and a PATCH addresses the resource it
            is sent to. A body that could name a second dump would be a body that can
            disagree with the path.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | RecorderDumpsUpdateResponse200
    """

    return (
        await asyncio_detailed(
            id=id,
            client=client,
            body=body,
        )
    ).parsed
