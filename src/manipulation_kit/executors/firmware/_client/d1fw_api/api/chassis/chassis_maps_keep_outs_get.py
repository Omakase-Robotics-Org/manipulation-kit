from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_maps_keep_outs_get_response_200 import (
    ChassisMapsKeepOutsGetResponse200,
)
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    scene: str,
) -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "get",
        "url": "/v1/chassis/maps/{scene}/keep_outs".format(
            scene=quote(str(scene), safe=""),
        ),
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisMapsKeepOutsGetResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisMapsKeepOutsGetResponse200.from_dict(response.json())

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
) -> Response[ChassisMapsKeepOutsGetResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisMapsKeepOutsGetResponse200 | ErrorEnvelope]:
    """Read one map's keep-out areas

     The areas the base refuses to plan through, forwarded unchanged, so the shape is the base's and is
    not pinned by this document. The base spells each area as a serialised JSON string inside
    `forbiddenAreasTemp`, with the structured special areas in `spliceAreas`.

    Args:
        scene (str):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisMapsKeepOutsGetResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        scene=scene,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
) -> ChassisMapsKeepOutsGetResponse200 | ErrorEnvelope | None:
    """Read one map's keep-out areas

     The areas the base refuses to plan through, forwarded unchanged, so the shape is the base's and is
    not pinned by this document. The base spells each area as a serialised JSON string inside
    `forbiddenAreasTemp`, with the structured special areas in `spliceAreas`.

    Args:
        scene (str):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisMapsKeepOutsGetResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        scene=scene,
        client=client,
    ).parsed


async def asyncio_detailed(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisMapsKeepOutsGetResponse200 | ErrorEnvelope]:
    """Read one map's keep-out areas

     The areas the base refuses to plan through, forwarded unchanged, so the shape is the base's and is
    not pinned by this document. The base spells each area as a serialised JSON string inside
    `forbiddenAreasTemp`, with the structured special areas in `spliceAreas`.

    Args:
        scene (str):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisMapsKeepOutsGetResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        scene=scene,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
) -> ChassisMapsKeepOutsGetResponse200 | ErrorEnvelope | None:
    """Read one map's keep-out areas

     The areas the base refuses to plan through, forwarded unchanged, so the shape is the base's and is
    not pinned by this document. The base spells each area as a serialised JSON string inside
    `forbiddenAreasTemp`, with the structured special areas in `spliceAreas`.

    Args:
        scene (str):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisMapsKeepOutsGetResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            scene=scene,
            client=client,
        )
    ).parsed
